import rospy
from std_srvs.srv import Empty

# maybe do some 'wait for service' here
reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

# invoke
reset_simulation()