#!/usr/bin/python
import rospy
import time
import numpy as np
from geometry_msgs.msg import Wrench

class FLCFilters(object):
	def __init__(self):
		'''Parameters Inicialization '''
		self.rospy = rospy
		self.frc_topic = self.rospy.get_param("frc_topic","/frc")
        self.acontroller_rate = self.rospy.get_param("acontroller_rate",6)
		self.controller_params = {
					"m": self.rospy.get_param("mass",15),
					"b_l": self.rospy.get_param("ldaming_ratio",5),
					"j": self.rospy.get_param("inertia",5),
					"b_a": self.rospy.get_param("adamping_ratio",4),
					"Ts": self.rospy.get_param("Ts",1.0/self.acontroller_rate)}
		'''Subscribers'''
		self.sub_frc = self.rospy.Subscriber(self.frc_topic,Wrench,self.callback_frc)
		#self.sub_trq = self.rospy.Subscriber(self.trq_topic, Wrench,self.callback_trq)
		'''Publishers'''
		self.pub_aux_cmd_vel = self.rospy.Publisher(self.aux_cmd_vel_topic, Twist, queue_size = 10)
		'''Node Configuration'''
		self.rospy.init_node("Admitance_Controller", anonymous = True)
		self.rate = self.rospy.Rate(self.acontroller_rate)
		self.vel = Twist()
		self.frc = 0
		self.trq = 0
		lnum = np.ones(2)*self.controller_params["Ts"]/2
		lden = [self.controller_params["m"]+self.controller_params["b_l"],self.controller_params["b_l"]-self.controller_params["m"]]
		anum = np.ones(2)*self.controller_params["Ts"]/2
		aden = [self.controller_params["j"]+self.controller_params["b_a"],self.controller_params["b_a"]-self.controller_params["j"]]
		self.systems = {
				"linear": sg.TransferFunction(lnum,lden,dt = self.controller_params["Ts"]/2),
				"angular": sg.TransferFunction(anum,aden,dt = self.controller_params["Ts"]/2)
				}
		#self.change = {"frc": False,
		#			   "trq": False}
		self.change = False
		self.signal_in = {"frc": [],
					  	  "trq": [],
					  	  "t": np.arange(0,1,self.controller_params["Ts"])}
		self.main_controller()
