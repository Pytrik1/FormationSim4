#!/usr/bin/env python
from __future__ import division
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation 
from matplotlib import style 
from platoon_merging.msg import data_agent
import numpy as np
from matplotlib.lines import Line2D
import math 
import time
import csv
from gazebo_msgs.srv import SetModelState, GetModelState, GetModelStateRequest
from gazebo_msgs.msg import ModelState, ModelStates
import platooning_agent_1_generationmu 
import platooning_agent_2_generationmu 
import platooning_agent_3_generationmu 
import platooning_agent_4_generationmu 

class process_data:
	def __init__(self):
		with open('/home/s2504111/monte_carlo_simulations/input/test2.csv') as p:
			reader = csv.reader(p, delimiter=',')
			next(reader)
			self.input = [map(float,rec) for rec in csv.reader(p)]
			self.n = len(self.input[0])
			print 'self.n', self.n

		self.i = 0

		self.header_n_1 = []
		self.error_x_n_1 =[]
		self.error_y_n_1 =[]
		self.linear_x_n_1 =[]
		self.linear_y_n_1 =[]
		self.d_t_n_1 = []
		self.distance_x_n_1 = []
		self.time_n_1 =[]
		self.header_n_2 = []
		self.error_x_n_2 =[]
		self.error_y_n_2 =[]
		self.linear_x_n_2 =[]
		self.linear_y_n_2 =[]
		self.d_t_n_2 = []
		self.distance_x_n_2 = []
		self.time_n_2 =[]
		self.header_n_3 = []
		self.error_x_n_3 =[]
		self.error_y_n_3 =[]
		self.linear_x_n_3 =[]
		self.linear_y_n_3 =[]
		self.d_t_n_3 = []
		self.time_n_3 =[]
		self.distance_x_n_3 = []
		self.header_n_4 = []
		self.error_x_n_4 =[]
		self.error_y_n_4 =[]
		self.linear_x_n_4 =[]
		self.linear_y_n_4 =[]
		self.d_t_n_4 = []
		self.distance_x_n_4 = []
		self.time_n_4 =[]
		self.running = True
		self.k_1 = 0
		self.k_2 = 0
		self.k_3 = 0
		self.k_4 = 0
		rospy.on_shutdown(self.shutdown)
		rospy.Subscriber('n_1/processed_data', data_agent, self.store_data_n_1)
		rospy.Subscriber('n_2/processed_data', data_agent, self.store_data_n_2)
		rospy.Subscriber('n_3/processed_data', data_agent, self.store_data_n_3)
		rospy.Subscriber('n_4/processed_data', data_agent, self.store_data_n_4)
		self.g_set_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)
		self.simulation()

	def simulation(self):
		print 'in simulation'
		while self.i < self.n:
			print 'in while loop'
			pose_n_1 = Pose(Point(self.input[i][0],self.input[i][1],0),Quaternion())
			state_n_1 = ModelState()
			state_n_1.pose = pose_n_1
			state_n_1.model_name = "nexus_1"
			state_n_1.reference_frame = "world"
			g_set_state(state_n_1)

	def store_data_n_1(self, msg):
		#print ' in store data n_1'
		if self.running:
			self.header_n_1 = msg.header
			self.error_x_n_1.append(msg.error_x)
			self.error_y_n_1.append(msg.error_y)
			self.linear_x_n_1.append(msg.linear_x)
			self.linear_y_n_1.append(msg.linear_y)
			self.time_n_1.append(msg.time)
			self.d_t_n_1.append(msg.dt)
			self.distance_x_n_1.append(msg.distance_x)
			length_time = len(self.time_n_1)
			#for i in range(len(self.distance_x_n_1)):
			#	if self.k_1==0 and self.distance_x_n_1[i] > 20:
			#		print 'at t = ',  self.time_n_1[i-1], 'self.distance_x_n_1 = ', self.distance_x_n_1[i-1]
			#		self.k_1 = 1
			for i in range(length_time):
				if self.time_n_1[i] > 15:
					print 'in if loop n_1'
					rospy.sleep(1)
					self.shutdown()

	def store_data_n_2(self, msg):
		#print ' in store data n_2'
		if self.running:
			self.header_n_2 = msg.header
			self.error_x_n_2.append(msg.error_x)
			self.error_y_n_2.append(msg.error_y)
			self.linear_x_n_2.append(msg.linear_x)
			self.linear_y_n_2.append(msg.linear_y)
			self.time_n_2.append(msg.time)
			self.d_t_n_2.append(msg.dt)
			self.distance_x_n_2.append(msg.distance_x)
			length_time = len(self.time_n_2)
			for i in range(len(self.distance_x_n_2)):
				if self.k_2==0 and self.distance_x_n_2[i] > 20: # Normal merging lanes in the NL are 180m. High speed is 180 km/h = 36 m/s. Use a gain of 9 becaue v = 4.
					print '----------------------------------------------'
					print 'at t = ',  self.time_n_2[i-1]
					print 'self.distance_x_n_2 = ', self.distance_x_n_2[i-1]
					print 'error_x_n_2 =', self.error_x_n_2[i-1]
					print 'error_y_n_2 =', self.error_y_n_2[i-1]
					print '----------------------------------------------'
					self.k_2 = 1

	def store_data_n_3(self, msg):
		if self.running:
			self.header_n_3 = msg.header
			self.error_x_n_3.append(msg.error_x)
			self.error_y_n_3.append(msg.error_y)
			self.linear_x_n_3.append(msg.linear_x)
			self.linear_y_n_3.append(msg.linear_y)
			self.time_n_3.append(msg.time)
			self.d_t_n_3.append(msg.dt)
			self.distance_x_n_3.append(msg.distance_x)
			length_time = len(self.time_n_3)


	def store_data_n_4(self, msg):
		if self.running:
			self.header_n_4 = msg.header
			self.error_x_n_4.append(msg.error_x)
			self.error_y_n_4.append(msg.error_y)
			self.linear_x_n_4.append(msg.linear_x)
			self.linear_y_n_4.append(msg.linear_y)
			self.time_n_4.append(msg.time)
			self.d_t_n_4.append(msg.dt)
			self.distance_x_n_4.append(msg.distance_x)
			length_time = len(self.time_n_4)
			#for i in range(len(self.distance_x_n_4)):
			#	if self.k_4==0 and self.distance_x_n_4[i] > 20:
			#		print 'at t = ',  self.time_n_4[i-1], 'self.distance_x_n_4 = ', self.distance_x_n_4[i-1]
			#		self.k_4 = 1

	# function that returns dxi / dt
	#def model(self, xi, t):
	#	mu = math.sin(math.pi/2)
	#	omega = math.pi 
	#	d_xi_d_t = np.array([(-1, -1, 0),(-1, -1, omega), (0, -omega, 0) ]) * xi + self.gain * np.transpose(self.R) * (self.error_x_n_2[-1] + mu)
	#	return d_xi_d_t


	def shutdown(self):
		rospy.loginfo("Start creating plots")
		t = self.time_n_2
		#xi_00 = np.array([(0), (0), (0)])
		#xi = odeint(self.model,xi_00, t)
		#length_xi = len(xi)
		#length_t = len(t)
		#print 'length_xi', length_xi, 'length_t', length_t
		#plt.figure(5)
		#plt.title('value of xi')
		#plt.plot(t,xi)
		#plt.show()
"""
		self.running = False
		plt.figure(0)
		plt.title('Error relative to i-1 in x-direction')
		plt.plot(self.time_n_1, self.error_x_n_1, label="e_x_n_1")
		plt.plot(self.time_n_2, self.error_x_n_2, label="e_x_n_2")
		plt.plot(self.time_n_3, self.error_x_n_3, label="e_x_n_3")
		plt.plot(self.time_n_4, self.error_x_n_4, label="e_x_n_4")
		plt.xlabel('time [s]')
		plt.ylabel('error x direction [m]')
		plt.legend()
		plt.savefig('/home/s2504111/catkin_ws/src/platoon_merging/figure/combined_error_x.png')
		plt.show()

		plt.figure(1)
		plt.title('Error relative to i-1 in y-direction')
		plt.plot(self.time_n_1, self.error_y_n_1, label="e_y_n_1")
		plt.plot(self.time_n_2, self.error_y_n_2, label="e_y_n_2")
		plt.plot(self.time_n_3, self.error_y_n_3, label="e_y_n_3")
		plt.plot(self.time_n_4, self.error_y_n_4, label="e_y_n_4")
		plt.ylabel('time [s]')
		plt.ylabel('error y direction [m]')
		plt.legend()
		plt.savefig('/home/s2504111/catkin_ws/src/platoon_merging/figure/combined_error_y.png')
		plt.show()

		plt.figure(2)
		plt.title('Linear velocity in x-direction')
		plt.plot(self.time_n_1, self.linear_x_n_1, label="v_x_n_1")
		plt.plot(self.time_n_2, self.linear_x_n_2, label="v_x_n_2")
		plt.plot(self.time_n_3, self.linear_x_n_3, label="v_x_n_3")
		plt.plot(self.time_n_4, self.linear_x_n_4, label="v_x_n_4")
		plt.ylabel('time [s]')
		plt.ylabel('velocity x direction [m/s]')
		plt.legend()
		plt.savefig('/home/s2504111/catkin_ws/src/platoon_merging/figure/linear_vel_x.png')
		plt.show()

		plt.figure(3)
		plt.title('Linear velocity in y-direction')
		plt.plot(self.time_n_1, self.linear_y_n_1, label="v_y_n_1")
		plt.plot(self.time_n_2, self.linear_y_n_2, label="v_y_n_2")
		plt.plot(self.time_n_3, self.linear_y_n_3, label="v_y_n_3")
		plt.plot(self.time_n_4, self.linear_y_n_4, label="v_y_n_4")
		plt.ylabel('time [s]')
		plt.ylabel('velocity y direction [m/s]')
		plt.legend()
		plt.savefig('/home/s2504111/catkin_ws/src/platoon_merging/figure/linear_vel_y.png')
		plt.show()

		plt.figure(4)
		plt.title('Driven distance agent n_4 in x-direction')
		#plt.plot(self.time_n_1, self.distance_x_n_1, label="d_x_n_1")
		#plt.plot(self.time_n_2, self.distance_x_n_2, label="d_x_n_2")
		#plt.plot(self.time_n_3, self.distance_x_n_3, label="d_x_n_3")
		plt.plot(self.time_n_4, self.distance_x_n_4, label="d_x_n_4")
		plt.ylabel('time [s]')
		plt.ylabel('distance in x -direction [m]')
		plt.legend()
		plt.savefig('/home/s2504111/catkin_ws/src/platoon_merging/figure/distance_x_direction.png')
		plt.show()
	"""



	
if __name__ == '__main__':
    rospy.init_node('creating_plots', anonymous=False)
    process_data()
    rospy.spin() 