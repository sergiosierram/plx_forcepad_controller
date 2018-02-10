#!/usr/bin/python
#EXcaliberPC
import rospy
import numpy as np
import scipy as sp
from geometry_msgs.msg import Twist,Wrench
from std_msgs.msg import Bool
from scipy import signal as sg
from sys import stdin

class AdmittanceController(object):
	def __init__(self):
		'''Parameters Inicialization '''
		self.rospy = rospy
		self.aux_cmd_vel_topic = self.rospy.get_param("aux_cmd_vel_topic", "/aux_cmd_vel")
		self.frc_topic = self.rospy.get_param("frc_topic","/linear_force")
		self.trq_topic = self.rospy.get_param("trq_topic","/torque")
		self.acontroller_rate = self.rospy.get_param("acontroller_rate",20)
		self.controller_params = {
									"m": self.rospy.get_param("mass",60),
									"b_l": self.rospy.get_param("ldamping_rario",0.5),
									"j": self.rospy.get_param("inertia",60),
									"b_a": self.rospy.get_param("adamping_rario",0.5),
									"Ts": self.rospy.get_param("Ts",1/self.acontroller_rate)
								 }
		'''Subscribers'''
		self.sub_frc = self.rospy.Subscriber(self.frc_topic, Wrench,self.callback_frc)
		self.sub_trq = self.rospy.Subscriber(self.trq_topic, Wrench,self.callback_trq)
		'''Publishers'''
		self.pub_aux_cmd_vel = self.rospy.Publisher(self.aux_cmd_vel_topic, Twist, queue_size = 10)
		'''Node Configuration'''
		self.rospy.init_node("Admitance Controller", anonymous = True)
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
		self.change = {
					   "frc": False,
					   "trq": False
					  }
		self.main_controller()

	def get_responce(self,systems,signal_in):
		signal_out["linear"] = sg.dlsim(systems["linear"],signal_in["frc"])#,signal_in["t"])
		signal_out["angular"] = sg.dlsim(systems["angular"],signal_in["trq"])#,signal_in["t"])
		return signal_out["linear"][-1],signal_out["angular"][-1]

	def callback_frc(self,msg):
		self.frc = msg.force.y
		self.change["frc"] = True
		return

	def callback_trq(self,msg):
		self.trq = msg.toque.y
		self.change["trq"] = True
		return

	def main_controller(self):
		signal_in = {
					  "frc": [],
					  "trq": [],
					  "t": np.arange(0,1,self.controller_params["Ts"])
					}
		while not(self.rospy.is_shutdown()) and len(in_signal_frc) < int(1/self.controller_params["Ts"]):
			if self.change["frc"]:
				signal_in["frc"].append(self.frc)
				self.change["frc"] = False
			if self.change["trq"]:
				signal_in["trq"].append(self.trq)
				self.change["trq"] = False
			self.rate.sleep()
		self.vel.linear.y,self.vel.angular.y = self.get_responce(self.systems, signal_in)
		self.pub_aux_cmd_vel.publish(self.vel)
		while not self.rospy.is_shutdown():
			if self.change["frc"]:
				signal_in["frc"].pop(0)
				signal_in["frc"].append(self.frc)
			if self.change["trq"]:
				signal_in["trq"].pop(0)
				signal_in["trq"].append(self.frc)
			if self.change["frc"] and self.change["trq"]:
				self.vel.linear.y,self.vel.angular.y = self.get_responce(self.systems, signal_in)
				self.pub_aux_cmd_vel.publish(self.vel)
				self.change["frc"] = False
				self.change["trq"] = False
			self.rate.sleep()

if __name__ == '__main__':
	try:
		ac = AdmittanceController()
	except rospy.ROSInterruptException:
		pass
