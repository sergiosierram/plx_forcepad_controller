#!/usr/bin/python
import rospy
import numpy as np
from geometry_msgs.msg import Wrench

class FLCFilters(object):
	def __init__(self):
		'''Parameters Inicialization '''
		self.rospy = rospy
		self.frc_topic = self.rospy.get_param("frc_topic","/frc1")
		self.frc_left_topic = self.rospy.get_param("frc_left","/frc_left")
		self.frc_right_topic = self.rospy.get_param("frc_right","/frc_right")
		self.wflc_params = {"M": self.rospy.get_param("wflc_order",4),
			"mu": self.rospy.get_param("wflc_amplitude_a_g",0.05),
			"mu0": self.rospy.get_param("wflc_frequency_a_g",0.001),
			"mub": self.rospy.get_param("wflc_bias_a_g",0.05),
			"sum_w0": 0,
			"w0": 0,
			"X": [],
			"W": [],
			"Wb": 0}
		self.r_flc_params = {"M": self.rospy.get_param("r_flc_order",3),
		     "mu": self.rospy.get_param("r_flc_amplitude_a_g",0.001),
		     "X": [],
		     "W": []}
		self.l_flc_params = {"M": self.rospy.get_param("l_flc_order",3),
		   	 "mu": self.rospy.get_param("l_flc_amplitude_a_g",0.001),
		   	 "X": [],
		   	 "W": []}
		'''Subscribers'''
		#self.sub_frc = self.rospy.Subscriber(self.frc_topic,Wrench,self.callback_frc)
		self.pub_left = self.rospy.Subscriber(self.frc_left_topic, Wrench, self.callback_frc_left)
		self.pub_right = self.rospy.Subscriber(self.frc_right_topic, Wrench, self.callback_frc_right)
		'''Publishers'''
		self.pub_frc = rospy.Publisher(self.frc_topic,Wrench, queue_size = 10)
		'''Node Configuration'''
		self.rospy.init_node("FLC_Filters", anonymous = True)
		self.frc = Wrench()
		self.frc_left = Wrench()
		self.frc_right = Wrench()
		self.change = {"left": False,
					   "right": False}
		self.wflc_params["W"] = np.random.rand(self.wflc_params["M"]*2)
		self.l_flc_params["W"] = np.random.rand(self.l_flc_params["M"]*2)
		self.r_flc_params["W"] = np.random.rand(self.r_flc_params["M"]*2)
		self.rospy.spin()

	def callback_frc_left(self,msg):
		self.frc_left = msg
		if self.change["right"]:
			self.main_callback()
		else:
			self.change["left"] = True
		return

	def callback_frc_right(self,msg):
		self.frc_right = msg
		if self.change["left"]:
			self.main_callback()
		else:
			self.change["right"] = True
		return

	def main_callback(self):
		fz = self.frc_left.force.z + self.frc_right.force.z
		w0,tremor = self.wflc(fz)
		print(w0)
		left_tremor,self.l_flc_params = self.flc(self.frc_left.force.y,w0/2,self.l_flc_params)
		right_tremor,self.r_flc_params = self.flc(self.frc_left.force.y,w0/2,self.r_flc_params)
		"""self.frc.force.y = tremor
		self.frc.force.x = tremor
		self.frc.force.z = fz"""
		self.frc_left.force.y = self.frc_left.force.y - left_tremor
		self.frc_right.force.y = self.frc_right.force.y - right_tremor
		self.frc.force.y = self.frc_left.force.y# + self.frc_right.force.y
		self.frc.torque.y = self.frc_left.force.y - self.frc_right.force.y
		self.pub_frc.publish(self.frc)
		self.change["left"] = False
		self.change["right"] = False

	def wflc(self,signal):
		self.wflc_params["X"] = []
		for r in range (0,self.wflc_params["M"]):
			self.wflc_params["X"].append(np.sin((r+1)*self.wflc_params["sum_w0"]))
			self.wflc_params["X"].append(np.cos((r+1)*self.wflc_params["sum_w0"]))

		y = np.dot(self.wflc_params["W"],self.wflc_params["X"])
		error = signal - y - self.wflc_params["Wb"]

		delta = 0
		for r in range (0,self.wflc_params["M"]):
			t1 = self.wflc_params["W"][r]*self.wflc_params["X"][self.wflc_params["M"]+r]
			t2 = self.wflc_params["W"][self.wflc_params["M"]+r]*self.wflc_params["X"][r]
			delta += (r+1)*(t1-t2)

		self.wflc_params["w0"] += 2*self.wflc_params["mu0"]*error*delta
		self.wflc_params["w0"] = abs(self.wflc_params["w0"])
		self.wflc_params["sum_w0"] += self.wflc_params["w0"]
		self.wflc_params["W"] += 2*self.wflc_params["mu"]*error*np.array(self.wflc_params["X"])
		self.wflc_params["Wb"] += 2*self.wflc_params["mub"]*error

		tremor = np.dot(self.wflc_params["W"],self.wflc_params["X"])
		#f0 = (self.wflc_params["w0"]/(2*np.pi)
		return self.wflc_params["w0"],tremor

	def flc(self,signal,w0,params):
		params["X"] = []
		for r in range (0,params["M"]):
			params["X"].append(np.sin((r+1)*w0))
			params["X"].append(np.cos((r+1)*w0))

		y = np.dot(params["W"],params["X"])
		error = signal - y

		params["W"] += 2*params["mu"]*error*np.array(params["X"])

		tremor = np.dot(params["W"],params["X"])

		return tremor,params


if __name__ == '__main__':
	try:
		flc_filters = FLCFilters()
	except rospy.ROSInterruptException:
		print("Something's gone wrong. Exiting")