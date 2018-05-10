#!/usr/bin/env python
import rospy
import socket
import numpy as np
import re
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String, Int8MultiArray
import Subsciber_generator

class RobotTracker:
  def __init__(self):
	rospy.init_node('Robot Tracking')
	# TCP_IP = rospy.get_param('~IP', 'localhost')
	# TCP_PORT = rospy.get_param('~PORT', 6666)
	# self.BUFFER_SIZE = 1024

	# self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	# self.s.connect((TCP_IP, TCP_PORT))

	self.validId = Int8MultiArray()
	self.f = open('/home/abhay/ROS_Projects/test/src/testpkg/scripts/data.txt', 'r')

	# Subsciber_generator.subscribe('/CT0/validity', Int8MultiArray, self.ct0_valid_callback)
	rospy.Subscriber('/CT0/validity', Int8MultiArray, self.ct0_valid_callback)

	self.location_pub = rospy.Publisher('~poses', String,queue_size=10)
	self.valid_id_pub = rospy.Publisher('~validity', Int8MultiArray, queue_size=10)

	self.rate = rospy.get_param('~rate', 10)
	self.test = Int8MultiArray()
	self.validity_ct0 = []
	self.valid_id_count_ct0 = 0

  def ct0_valid_callback(self, msg, name):
  	rospy.loginfo('inside callback '+name)
	test1 = []
	self.test = msg.data
	self.valid_id_count_ct0 = len(self.test)
	for i in range(self.valid_id_count_ct0):
		test1.append(msg.data[i])
	self.validity_ct0 = test1
	print 'msg.data', msg.data
	print 'test1', test1
	print '------------------'

  def update(self):
	# data = self.s.recv(self.BUFFER_SIZE)
	x = self.f.readline()
	data = ''
	while(x != '\n'):
		data = data + x
		x = self.f.readline()
		# rospy.loginfo(str(x == '\n'))
	
	# rospy.loginfo(data)

	self.location_pub.publish(data)
	
	# Find all the Id's present in the frame
	id_array = np.full( (50), -1) # max array size is limited to 25 (50/2) (string length of ID's)
	loop = 0
	# find the Id's
	for m in re.finditer('ValidID', data):
		id_array[loop] = data[m.end()+1:m.end()+3]          
		loop = loop + 1

	# find unique Ids and remove -1
	unique_ids = np.unique(id_array)
	ind = np.argwhere(unique_ids==-1)
	unique_ids = np.delete(unique_ids, ind)

	# -----------------------------------------------------------------------------
	# if robot is in frames of multiple cameras then we should only take one value.
	# -----------------------------------------------------------------------------

	intersection = list(set(self.validity_ct0).intersection(unique_ids))
	for loop in range(len(intersection)):
		ind = np.argwhere(unique_ids==intersection[loop])
		unique_ids = np.delete(unique_ids, ind)
	print(unique_ids)
	# publish the valid Id's present in the camera frame
	self.validId.data = unique_ids
	self.valid_id_pub.publish(self.validId)
	
	#parse to extract data of each robot  
	Array_Robots= [x for x in data.split('\n') if x!='']


	for index in range(len(Array_Robots) - 2):
		val =-1
		for m in re.finditer('ValidID', Array_Robots[index+1]):
			val = int(Array_Robots[index+1][m.end()+1 : m.end()+3])         
		if val >= 0:
			Array_All = Array_Robots[index+1].split()
			if Array_All[0] == "Robot":
				Topic_Name = '~robot' + str(val)
				pub1 = rospy.Publisher(Topic_Name, Pose2D, queue_size=10)
				X = float(Array_All[2])
				Y = float(Array_All[3])
				Theta = float(Array_All[4])
				data1 =Pose2D(X, Y, Theta)
				pub1.publish(data1)
	
  def spin(self):
	rospy.loginfo("Start Robot Tracking for camera 1")
	rate = rospy.Rate(self.rate)
	rospy.on_shutdown(self.shutdown)
	while not rospy.is_shutdown():
	  self.update();
	  rate.sleep()
	rospy.spin()

  def shutdown(self):
	rospy.loginfo("Stop Robot Tracking for camera 1")
	rospy.sleep(1)

def main():
  Robot_Tracker = RobotTracker();
  Robot_Tracker.spin()

if __name__ == '__main__':
  main(); 
