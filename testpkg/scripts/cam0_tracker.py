#!/usr/bin/env python
import rospy
import socket
import numpy as np
import re
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String, Int8MultiArray

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


	self.location_pub = rospy.Publisher('~poses', String,queue_size=10)
	self.valid_id_pub = rospy.Publisher('~validity', Int8MultiArray, queue_size=10)

	self.rate = rospy.get_param('~rate', 10)

  def update(self):
	# data = self.s.recv(self.BUFFER_SIZE)
	x = self.f.readline()
	data = ''
	while(x != '\n'):
		data = data + x
		x = self.f.readline()

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

	# publish the valid Id's present in the camera frame
	self.validId.data = unique_ids
	self.valid_id_pub.publish(self.validId)
	
	#parse to extract data of each robot  
	Array_Robots= data.split('\n')

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
	rospy.loginfo("Start Robot Tracking for camera 0")
	rate = rospy.Rate(self.rate)
	rospy.on_shutdown(self.shutdown)
	while not rospy.is_shutdown():
	  self.update();
	  rate.sleep()
	rospy.spin()

  def shutdown(self):
	rospy.loginfo("Stop Robot Tracking for camera 0")
	rospy.sleep(1)

def main():
  Robot_Tracker = RobotTracker();
  Robot_Tracker.spin()

if __name__ == '__main__':
  main(); 
