#!/usr/bin/python
import rospy
import time
from threading import Thread
from geometry_msgs.msg import Wrench
from sys import stdin
from daq_controller.DAQ import DAQ

class FrcAcquirer():
	def __init__(self):
		self.pub_left = rospy.Publisher('frc_left', Wrench, queue_size = 10)
		self.pub_right = rospy.Publisher('frc_right', Wrench, queue_size = 10)
		self.pub_trq = rospy.Publisher('torque',Wrench, queue_size = 10)
		self.frc_left = Wrench()
		self.frc_right = Wrench()
		self.trq = Wrench()
		self.exitFlag = False
		self.fs = 10
		self.daq = DAQ()
		self.x_forces = False
		self.rospy = rospy
		self.rospy.init_node('frc_acquisition', anonymous = True)
		self.rate = self.rospy.Rate(self.fs)
		self.dataExport = [False, 'data.txt']

	def acquire(self):
		if self.dataExport[0]:
			f = open(self.dataExport[1],'w')
			ti = time.time()
		while not self.exitFlag and not rospy.is_shutdown():
			data = [float(i) for i in self.daq.get_forces(self.x_forces).split('\t')]
			if self.x_forces:
				self.frc_left.force.x, self.frc_left.force.y, self.frc_left.force.z = data[0], data[1], data[2]
				self.frc_right.force.x, self.frc_right.force.y, self.frc_right.force.z = data[3], data[4], data[5]
				self.trq.torque.x = float(data[3])-float(data[0])
				self.trq.torque.y = float(data[4])-float(data[1])
				self.trq.torque.z = float(data[5])-float(data[2])
			else:
				self.frc_left.force.x = 0
				self.frc_right.force.x = 0
				self.frc_left.force.y, self.frc_left.force.z = data[0], data[1]
				self.frc_right.force.y, self.frc_right.force.z = data[2], data[3]
				self.trq.torque.x = 0
				self.trq.torque.y = float(data[2])-float(data[0])
				self.trq.torque.z = float(data[3])-float(data[1])
			if self.dataExport[0]:
				t = round(time.time()-ti,2)
				f.write(str(t)+" "+" ".join(map(str, data))+str("\n"))
			self.pub_left.publish(self.frc_left)
			self.pub_right.publish(self.frc_right)
			self.pub_trq.publish(self.trq)
			self.rate.sleep()
		print("Exiting ...")
		if self.dataExport[0]:
			f.close()
		self.serialPort.close()

	def wait_for_exit(self):
		print("Press Enter for exit: ")
		x = stdin.readline()
		self.exitFlag = True

if __name__ == '__main__':
	try:
		frc = FrcAcquirer()
		thread1 = Thread(target = frc.wait_for_exit)
		thread1.start()
		frc.acquire()
	except rospy.ROSInterruptException:
		print("Something's gone wrong. Exiting")
