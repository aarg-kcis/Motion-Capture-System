#!/usr/bin/env python
import sys
import rospy
import json
import socket
import threading
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String, Int8MultiArray
from custom_msgs.msg import CamPoseArray

class CamTracker:
	def __init__(self, idd, num_bots, tcp_ip, tcp_port, buffer_size):
		self.id = idd
		self.NUM_BOTS = num_bots
		self.BUFFER_SIZE = buffer_size
		self.NAME = "CT{:d}/".format(self.id) 
		self.rate = rospy.get_param('~rate', 10)
		self.poses = {}
		self.purana_pose = {}
		self.stop_thread = False
		self.thread = threading.Thread(target=self.run, args=(tcp_ip, tcp_port))
		self.campose_pub = rospy.Publisher(self.NAME+'pose', String, queue_size=10)

	def stop(self):
		msg = "Exitting thread for cam{:d}".format(self.id)
		rospy.loginfo(msg)
		self.stop_thread = True

	def connect_to_socket(self, ip, port):
		try:
			self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.s.connect((ip, port))
		except:
			msg = "Can't connect to given socket ({:d})'\
					' for CAM{:d}.".format(port, self.id)
			rospy.loginfo(msg)
			self.stop()
			self.s = None

	def init_poses(self):
		init_pose = {'x':0, 'y': 0, 'theta': 0}
		bot_list = range(self.NUM_BOTS)
		poses_list = [init_pose for x in bot_list]
		self.poses = {k:v for (k,v) in zip(bot_list, poses_list)}  

	def gen_pose(self, data_bits):
		init_pose = {'x':0, 'y': 0, 'theta': 0}
		init_pose['x'] = float(data_bits[0])
		init_pose['y'] = float(data_bits[1])
		init_pose['theta'] = float(data_bits[2])
		return init_pose
		

	def filter_contents(self):
		filtered_data = self.data.split('\n')
		idx = []
		for i, s in enumerate(filtered_data):
			if 'Detected' in s:
				idx.append(i)
				continue
			if len(idx) == 2:
				break
		if len(idx) < 2:
			return None
		filtered_data = filtered_data[idx[0]+1:idx[1]]
		self.data = filtered_data

	def get_valid_locations(self):
		for x in self.data:
			data_bits = x.split(' ')
			valid = int(data_bits[7])
			if valid >= 0:
				robot_id = int(data_bits[1])
				if valid == robot_id:
					self.poses[robot_id] = self.gen_pose(data_bits[2:5])
	
	def update(self):
		self.poses = {}
		if self.s != None:
			self.data = self.s.recv(self.BUFFER_SIZE)
			self.filter_contents()
			try:
				self.get_valid_locations()
			except:
				pass
			self.purana_pose = self.poses
			self.campose_pub.publish(json.dumps(self.purana_pose))

	def spin(self):
		rate = rospy.Rate(self.rate)
		rospy.on_shutdown(self.shutdown)
		while not (rospy.is_shutdown() or self.stop_thread):
			if self.stop_thread:
				break
			self.update();
			rate.sleep()
		rospy.spin()
	
	def shutdown(self):
		rospy.loginfo("Shutting down tracker for camera "+str(self.id))
		rospy.sleep(1)

	def run(self, tcp_ip, tcp_port):
		rospy.loginfo("Starting tracker for camera "+str(self.id))
		self.connect_to_socket(tcp_ip, tcp_port)
		self.spin()

