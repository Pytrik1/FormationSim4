#!/usr/bin/env python


from __future__ import division

from math import * 
import math
import rospy
import numpy as np
from rospy_tutorials.msg import Floats				
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
import matplotlib.pyplot as pl
import matplotlib.pyplot as plt
import rospy
import std_msgs
import sys
from platoon_merging.msg import data_agent
import time
from scipy.integrate import odeint 
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class process_laservalues:
	def __init__(self,speed, desired_x_distance,activate_mu):
		#print 'in init'
		self.name = 'n_3'
		self.min_range = -6
		self.max_range = 6
		self.speed = speed #in + x - direction
		self.desired_x_distance = speed*desired_x_distance
		self.error_margin_x = self.desired_x_distance * 0.1
		self.error_margin_y = 0.02
		self.running = True
		rospy.on_shutdown(self.shutdown)
		rospy.Subscriber(self.name + '/hokuyo_points', LaserScan, self.calculate_z_values)
		self.pub = rospy.Publisher(self.name +'/cmd_vel', Twist, queue_size=1)
		self.pub_data = rospy.Publisher(self.name +'/processed_data', data_agent, queue_size=1)
		self.velocity = Twist()
		self.velocity.linear.x = self.speed
		self.velocity.linear.y = 0
		self.pub.publish(self.velocity)
		self.error_x_direction_log = []
		self.error_y_direction_log = []
		self.store_linear_velocity_x = []
		self.store_linear_velocity_y = []
		self.msg_data=data_agent()
		self.k = 0
		self.temp_distance_d_t_vector = []
		self.sum_distance_d_t = []


		# initial conditions for mu 
		self.w_init = np.array([(1), (1), (1)])
		self.omega = math.pi
		self.xi_init =np.array([(1), (1), (1)])
		self.R = np.array([(1), (1), (0)])
		self.gain = 1 #GAIN

		# initial conditions for mu_head
		self.z_0 = np.array([(0),(0),(0)])

		self.g_set_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)

		# ------------------ NEW INITIAL CONDITIONS COMPACT FORM ----------------------- # 
		self.merging_agent = 0
		self.merging_agent_mu = 0 
		self.store_e_y = []
		self.store_e_x_pos = []
		self.store_e_x_neg = []
		self.store_e_x_pos_mu = []
		self.store_e_x_neg_mu = []
		self.activate_mu = activate_mu
		self.z_0 = [1,1,1]
		self.merge = 0

		self.mu_log = []
		self.mu_hat_log = []
		self.scale = 1

		self.now = np.float64([rospy.get_time()])
		self.old = np.float64([rospy.get_time()])
		self.begin = np.float64([rospy.get_time()]) 
		self.time = np.float64([])
		self.time_log = np.array([])
		self.d_t = np.float64([])

		self.e_x_neg_log = []
		self.e_x_neg_log_mu = []
		self.e_x_neg_log_mu_hat =[]

	def calculate_z_values(self, msg):
		#print 'in calculate_z_values'
		scandata= np.asarray(msg.ranges)
		store_angle =[]
		store_distance = []
		store_x_distance =[]
		store_y_distance = []
		if self.running: 
			# Erase datapoints from laser that are outside the ranges
		        for i in range(len(scandata)): #length of scandata is 360. 
		            if (scandata[i] < self.min_range) or (scandata[i] > self.max_range) or (scandata[i]<0.2): #scandata < 0.2 to filter data bugs. This is within the lengths of the nexus robot. 
		                scandata[i] = 0

			# Create three arrays for angle and x- and y- distance
			for i in range(len(scandata)):
				if scandata[i] != 0:
					store_angle.append(i)
					x_direction = np.float64(sin(radians(i))*scandata[i])
					y_direction = np.float64(-cos(radians(i))*scandata[i])
					store_x_distance.append(x_direction)
					store_y_distance.append(y_direction)

			length = len(store_x_distance)

			#print 'store_angle', self.name, store_angle
			#print 'store_x_distance', self.name, store_x_distance
			#print 'store_y_distance', self.name, store_y_distance
			self.error(store_x_distance, store_y_distance, store_angle, length)
	
	def error(self, store_x_distance, store_y_distance, store_angle,length):
		#print 'in error'
		# ------------------- INITIALIZE -------------------------------------------------------------#
		e_y_vec =[]
		e_x_pos_vec=[]
		e_x_neg_vec = []
		# ------------------- START TIME AFTER ACTIVATION VIA LASERSCAN DATA INPUT  ------------------#
		if self.k < 1:
			self.begin = np.float64([rospy.get_time()])
			self.k = 10
			self.time = 0

		# -------------------- CALCULATE ERROR VALUES ------------------------------------------------# 
		if length == 0: # no data present
			e_x_pos = 0
			e_x_neg = 0
			e_y = 0
			self.merging_agent = 0

		elif length != 0: # data is present 
				for i in range(length):

					if store_y_distance[i] > - self.error_margin_y and store_x_distance[i]>0:
						self.merging_agent = 1
						e_y_vec.append(store_y_distance[i])
					elif store_y_distance[i] <= 0:
						e_y_vec.append(self.min_range)
						self.merging_agent = 0
					else:
						e_y_vec.append(self.min_range)

					if store_x_distance[i] > self.error_margin_x:
						e_x_pos_vec.append(store_x_distance[i]-self.desired_x_distance)

					elif abs(store_x_distance[i]) < self.error_margin_x and store_y_distance[i] > 0:
						#print 'too close', self.name, store_x_distance[i], store_y_distance[i]
						self.controller(self.speed*0.1,0)

					elif store_x_distance[i] <= -self.error_margin_x:
						e_x_neg_vec.append(store_x_distance[i]+self.desired_x_distance)
						e_y_vec.append(self.min_range)

				#print 'MERGING AGENT BEFORE ', self.name, self.merging_agent
					#if store_x_distance[i] > 0:
					#	e_x_pos_vec.append(store_x_distance[i]-self.desired_x_distance)

					#elif store_x_distance[i] <= 0:
					#	e_x_neg_vec.append(store_x_distance[i]+self.desired_x_distance)
					#	e_y_vec.append(self.min_range)


				#print 'store_y_distance', store_y_distance
				e_y = max(e_y_vec)
				if e_y == self.min_range:
					e_y = 0
				if e_y > -self.error_margin_y:
					self.merging_agent = 1
				elif e_y <= - self.error_margin_y or e_y ==0:
					self.merging_agent = 0

				if e_y > self.error_margin_y:
					self.merging_agent_mu = 1
				else:
					self.merging_agent_mu = 0

				if len(e_x_pos_vec) == 0:
					e_x_pos = 0
				else: 
					e_x_pos = min(e_x_pos_vec)
				if len(e_x_neg_vec) == 0:
					e_x_neg = 0
				else:
					e_x_neg = max(e_x_neg_vec)

		self.store_e_y.append(e_y)
		self.store_e_x_pos.append(e_x_pos)
		self.store_e_x_neg.append(e_x_neg)
		self.now = np.float64([rospy.get_time()])

		if self.activate_mu == 1:
			if self.merging_agent_mu == 1 and self.time != 0 and e_x_neg != 0:
				self.generation_mu(e_y,e_x_pos,e_x_neg)
			else:
				self.conditions(e_y,e_x_pos,e_x_neg, 0, 0)
				self.msg_data.mu = []
				self.msg_data.mu_hat = []
		else:
			self.conditions(e_y,e_x_pos,e_x_neg, 0, 0)

	def generation_mu(self, e_y,e_x_pos,e_x_neg):
		# --------------- GENERATION OF DISAGREEMENT MU IN MEASUREMENT -------------------------- #
		#self.now = np.float64([rospy.get_time()])
		t = np.float64([self.now-self.begin])
		#print 't', t
		w_2 = np.float64(cos(self.omega*t)*self.w_init[1]+ sin(self.omega*t)*self.w_init[2])
		w_3 = np.float64(-sin(self.omega*t)*self.w_init[1]+ cos(self.omega*t)*self.w_init[2])
		w = [np.float64(self.w_init[0]), w_2, w_3]
		mu = self.scale *( self.R[0] * w[0] + self.R[1] * w[1] + self.R[2] * w[2])
		#print 'mu', mu
		self.mu_log.append(mu)
		self.e_x_neg_log.append(e_x_neg)
		e_x_neg_new = e_x_neg + mu
		#e_x_neg = e_x_neg + mu
		self.e_x_neg_log_mu.append(e_x_neg_new)
		
		#self.store_e_x_neg_mu.append(e_x_neg)


		# ----------------------- GENERATION OF ESTIMATOR MU_HEAD -------------------------------- #
		t_mu_hat = np.linspace(0,np.float64(self.time),20)
		#print 't_mu_hat', t_mu_hat
		n = len(t_mu_hat)
		#print 'n', n
		xi_1 = np.zeros_like(t_mu_hat)
		xi_2 = np.zeros_like(t_mu_hat)
		xi_3 = np.zeros_like(t_mu_hat)
		xi_1[0] = self.z_0[0]
		xi_2[0] = self.z_0[1]
		xi_3[0] = self.z_0[2]
		for i in range(1,n):
				new_z_0 = self.z_0
				tspan = ([t_mu_hat[i-1], t_mu_hat[i]])
				#print 'tspan', tspan 
				#print 'arguments odeint, new_z_0, tspan, e_x_neg_new', new_z_0, tspan, e_x_neg_new
				z_pos = odeint(self.model,new_z_0,tspan,args=(e_x_neg_new,))
				#print 'z_pos', z_pos
				xi_1[i] = z_pos[1][0]
				xi_2[i] = z_pos[1][1]
				xi_3[i] = z_pos[1][2]
				#print 'xi_1', xi_1
				self.z_0 = z_pos[1]

				if i == n-1:
					mu_hat = xi_1[-1] + xi_2[-1] # R*xi
					#print 'mu_hat', mu_hat
					e_x_neg_mu_hat = e_x_neg_new - mu_hat
					self.e_x_neg_log_mu_hat.append(e_x_neg_mu_hat)
					self.conditions(e_y,e_x_pos,e_x_neg_mu_hat, mu, mu_hat) #HIER MOET WAT VERANDEREN WANT DE E_X_POS WORDT NU ANDERS OPGESLAGEN DUS DE PLOTS WERKEN NIET MEER
					self.mu_hat_log.append(mu_hat)
					self.msg_data.mu = self.mu_log
					self.msg_data.mu_hat = self.mu_hat_log

	def model(self,z,t,e_x):
		xi_1 = z[0]
		xi_2 = z[1]
		xi_3 = z[2]
		dxi_1dt = -xi_1 - xi_2 + self.gain * (e_x)
		dxi_2dt = -xi_1 - xi_2 + self.omega * xi_3 + self.gain * (e_x)
		dxi_3dt = - self.omega * xi_2
		dzdt = [dxi_1dt,dxi_2dt, dxi_3dt]
		return dzdt

	def conditions(self,e_y,e_x_pos,e_x_neg, mu, mu_hat):
		# Only merge if the desired distance to the postive- and negative direction is achieved
		#e_x_pos = e_x_pos + mu - mu_hat
		if self.merging_agent == 1:
			if e_x_pos > -self.error_margin_x and e_x_neg < self.error_margin_x or self.merging_agent_mu == 1:
				#print 'safe! start merging'
				self.merge = 1
			#elif e_y > -self.error_margin_y:
			#	self.merge = 1
			else:
				self.merge = 0
				#print 'not safe!!!! do not merge!'

		#print 'self.merge', self.name, self.merge
		#print 'self.merging agent', self.name, self.merging_agent
		# Emergency stop 
		if self.speed + e_x_pos < 0 or e_x_pos < (-self.desired_x_distance+self.error_margin_x):
			stop = 0 
		else:
			stop = 1
		#print 'stop', self.name, stop
		#print 'e_y', self.name, e_y
		#print 'AAAAAAAAAAAAAAAAAAAAAAAA stop', self.name, stop
		#e_x_pos = e_x_pos - self.scale * mu_hat
		#e_x_neg = e_x_neg - self.scale * mu_hat
		u_x = (self.speed + e_x_pos)*stop 
		u_y = self.merging_agent*e_y*self.merge #altijd 0 als er geen data of het niet de 'merging agent is'

		self.msg_data.error_x = e_x_pos
		self.msg_data.error_x_neg = e_x_neg
		self.msg_data.error_y = e_y

		self.controller(u_x,u_y)

	def controller(self,u_x,u_y):
		#print 'in controller'
		# ------------------ GIVE VELOCITY COMMANDS --------------------------------------#
		if self.running:
			self.velocity.linear.x = u_x
			self.velocity.linear.y = u_y
			self.pub.publish(self.velocity)
			#print 'self.velocity', self.name, self.velocity


		# ---------------------- PREPARE DATA FOR MSG --------------------------------#
		# TIME 
		
		self.time = np.float64([self.now-self.begin])
		self.time_log = np.append(self.time_log, self.time)
		self.old = self.now 
		#print 'time', self.name, self.time

		length_time = int(len(self.time_log)-1)
		if length_time > 1:
			self.d_t = self.time_log[length_time]-self.time_log[(length_time-1)]
		else:
			self.d_t = 0

		#print 'dt', self.d_t
		# DISTANCE 
		temp_distance= self.d_t * self.velocity.linear.x
		self.temp_distance_d_t_vector.append(temp_distance)
		self.sum_distance_d_t = sum(self.temp_distance_d_t_vector)

		# MESSAGE
		self.msg_data.linear_x = self.velocity.linear.x
		self.msg_data.linear_y = self.velocity.linear.y
		self.msg_data.dt = self.d_t
		self.msg_data.distance_x = self.sum_distance_d_t
		self.msg_data.time = self.time
		self.pub_data.publish(self.msg_data)

	def shutdown(self):
		rospy.loginfo("Stopping")
		self.running = False
		self.velocity=Twist()
		self.pub.publish(self.velocity)
		self.data = data_agent()
		self.pub_data.publish(self.data)
"""
		x_axis = np.linspace(0,self.time_log[-1], len(self.mu_log))
		plt.figure(1)
		plt.title('Convergence of mu and mu_hat')
		plt.plot(x_axis,  self.mu_log, label="mu", color='r')
		plt.plot(x_axis, self.mu_hat_log, label="mu_hat", color='b')
		plt.xlabel('time steps')
		plt.ylabel('mu')
		plt.legend()
		plt.show()
"""


if __name__ == '__main__':
	rospy.init_node('platooning_agent_n_3_generationmu', anonymous=False) 
	speed = 4
	desired_x_distance = 0.5
	activate_mu = 0
	process_laservalues(speed,desired_x_distance,activate_mu)
	rospy.spin() 
