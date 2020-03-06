#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():
    
    finalpose_x = 5.0
    finalpose_y = 15.0

    client1 = actionlib.SimpleActionClient('/n_1/move_base',MoveBaseAction)
    client1.wait_for_server()

    client2 = actionlib.SimpleActionClient('/n_2/move_base',MoveBaseAction)
    client2.wait_for_server()

    client3 = actionlib.SimpleActionClient('/n_3/move_base',MoveBaseAction)
    client3.wait_for_server()

    client4 = actionlib.SimpleActionClient('/n_4/move_base',MoveBaseAction)
    client4.wait_for_server()

    goal1 = MoveBaseGoal()
    goal2 = MoveBaseGoal()
    goal3 = MoveBaseGoal()
    goal4 = MoveBaseGoal()

    goal1.target_pose.header.frame_id = "n_1_laser_frame"
    goal1.target_pose.header.stamp = rospy.Time.now()
    goal1.target_pose.pose.position.x = finalpose_x
    goal1.target_pose.pose.position.y = finalpose_y
    goal1.target_pose.pose.orientation.w = 1.0

    goal2.target_pose.header.frame_id = "n_2_laser_frame"
    goal2.target_pose.header.stamp = rospy.Time.now()
    goal2.target_pose.pose.position.x = finalpose_x
    goal2.target_pose.pose.position.y = finalpose_y
    goal2.target_pose.pose.orientation.w = 1.0

    goal3.target_pose.header.frame_id = "n_3_laser_frame"
    goal3.target_pose.header.stamp = rospy.Time.now()
    goal3.target_pose.pose.position.x = finalpose_x
    goal3.target_pose.pose.position.y = finalpose_y
    goal3.target_pose.pose.orientation.w = 1.0

    goal4.target_pose.header.frame_id = "n_4_laser_frame"
    goal4.target_pose.header.stamp = rospy.Time.now()
    goal4.target_pose.pose.position.x = finalpose_x
    goal4.target_pose.pose.position.y = finalpose_y
    goal4.target_pose.pose.orientation.w = 1.0

    client1.send_goal(goal1)
    client2.send_goal(goal2)
    client3.send_goal(goal3)
    client4.send_goal(goal4)

    wait1 = client1.wait_for_result()
    wait2 = client2.wait_for_result()
    wait3 = client3.wait_for_result()
    wait4 = client4.wait_for_result()

    if not wait1:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client1.get_result(), client2.get_result(), client3.get_result(), client4.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
