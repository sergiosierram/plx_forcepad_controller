#!/usr/bin/python
import rospy
import time
import scipy.signal as signal
import numpy as np
from threading import Thread
from geometry_msgs.msg import Wrench
from sys import stdin
from daq_controller.DAQ import DAQ

class FrcAcquirer(object):
    def __init__(self):
        '''ROS Parameters Inicialization '''
        self.rospy = rospy
        self.lfrc_topic = self.rospy.get_param("frc_topic","/frc_left")
        self.rfrc_topic = self.rospy.get_param("trq_topic","/frc_right")
        self.frc_topic = self.rospy.get_param("frc_topic","/linear_frc")
        self.trq_topic = self.rospy.get_param("trq_topic","/torque")
        self.frc_acquirer_rate = self.rospy.get_param("frc_acquirer_rate",100)
        self.filt_signals = self.rospy.get_param("filt_signals",True)
        '''Publishers'''
        self.pub_lfrc = rospy.Publisher(self.lfrc_topic, Wrench, queue_size = 10)
        self.pub_rfrc = rospy.Publisher(self.rfrc_topic, Wrench, queue_size = 10)
        self.pub_trq = rospy.Publisher(self.trq_topic,Wrench, queue_size = 10)
        self.pub_frc = rospy.Publisher(self.frc_topic,Wrench, queue_size = 10)
        '''Subscribers'''
        #This node don't have Subscribers
        '''Node Configuration'''
        self.frc_left = Wrench()
        self.frc_right = Wrench()
        self.trq = Wrench()
        self.frc = Wrench()
        self.exitFlag = False
        #self.daq = DAQ()
        if self.filt_signals:
            self.filter = {"N": 4,
            			   "fn": 5,
            			   "Wn": 2*self.filter["fn"]/self.fs,
            			   "a": [],
            			   "b": [],
                           "padlen": 0}
            a,b = signal.butter(self.filter["N"],self.filter["Wn"])
            self.filter["a"],self.filter["b"] = a,b
            self.filter["padlen"] = 3*max(len(self.filter["a"]),len(self.filter["b"]))
        self.rate = self.rospy.Rate(self.acontroller_rate)
        self.rospy.init_node(self.frc_acquirer_rate, anonymous = True)

    def wait_for_exit(self):
        print("Press Enter for exit: ")
        x = stdin.readline().strip()
        self.exitFlag = Truedata = [float(i) for i in self.daq.get_forces(self.x_forces).split('\t')]

    def publish_data(self,data):
        self.frc_left.force.x = 0
        self.frc_right.force.x = 0
        self.frc_left.force.y, self.frc_left.force.z = data[-1][2], data[-1][3]
        self.frc_right.force.y, self.frc_right.force.z = data[-1][0], data[-1][1]
        self.trq.torque.x = 0
        self.trq.torque.y = float(data[-1][0])-float(data[-1][2])
        self.trq.torque.z = float(data[-1][1])-float(data[-1][3])
        self.l_frc.force.x = 0
        self.l_frc.force.y = float(data[-1][0])+float(data[-1][2])
        self.l_frc.force.z = float(data[-1][1])+float(data[-1][3])
        self.pub_left.publish(self.frc_left)
        self.pub_right.publish(self.frc_right)
        self.pub_trq.publish(self.trq)
        self.pub_lfrc.publish(self.l_frc)

    def acquire(self):data = [float(i) for i in self.daq.get_forces(self.x_forces).split('\t')]
		buff = []
        while not rospy.is_shutdown() and self.filt_signals:
            buff.append([float(i) for i in self.daq.get_forces(self.x_forces).split('\t')])
            if len(buff) <= self.filter["padlen"]:
                breakdata = [float(i) for i in self.daq.get_forces(self.x_forces).split('\t')]
		while not self.exitFlag and not rospy.is_shutdown():
			data = [float(i) for i in self.daq.get_forces(self.x_forces).split('\t')]
            if self.filt_signals:
                buff.pop(0)
                buff.append(data)
                buff_filt = signal.filtfilt(b,a,buff,axis=0)
                self.publish_data(buff_filt)
            else:
                buff.append(data)
                self.publish_data(buff)
                buff = []
			self.rate.sleep()
		print("Exiting ...")
		self.daq.close_port()

if __name__ == '__main__':
    try:
        frc = FrcAcquirer()
        thread1 = Thread(target = frc.wait_for_exit)
        thread1.start()
        frc.acquire()
    except rospy.ROSInterruptException:
        print("Something's gone wrong. Exiting")
