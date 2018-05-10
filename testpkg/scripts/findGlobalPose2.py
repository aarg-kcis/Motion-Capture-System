#!/usr/bin/env python
import tf
import rospy
import numpy as np
from tfWrapper import  TFWrapper

from nav_msgs.msg import Odometry
from custom_msgs.msg import  CamPose, CamPoseArray
from geometry_msgs.msg import Point, Quaternion, Pose2D

class Robot:
	def __init__(self, id, tfs, publish=True):
		self.id = id
		self.tfs = tfs
		self.cam_id = None
		self.local_pose = {'x': None, 'y': None, 'th': None}
		self.global_pose = {'x': None, 'y': None, 'th': None}
		self.publish = publish
		if self.publish:
			self.init_publisher()

	def set_pose(self, data, cam_id):
		# This  has been done to as the coordinate frame of cosphi 
		# is different from ours. 
		self.local_pose['x'] = data.y
		self.local_pose['y'] = data.x
		self.local_pose['th'] = -data.theta
		self.cam_id = cam_id

	def gen_global_pose(self, mother_cam):
		if self.cam_id == mother_cam:
			self.global_pose = self.local_pose
			return
		self.TF = self.tfs[self.cam_id]
		C =	tf.transformations.compose_matrix(angles=(0,0,self.local_pose['th']) \
			,translate=(self.local_pose['x'], self.local_pose['y'], 0)) 
		D =	tf.transformations.concatenate_matrices(self.TF, C) 
		sc, sh, an, tr, pr = tf.transformations.decompose_matrix(D) 
		self.global_pose = {'x': tr[0], 'y': tr[1], 'th': an[2]}

	def publish_odom(self):
		if not self.publish:
			return
		msg = Odometry()
		msg.header.stamp = rospy.Time.now()
		msg.pose.pose.position = Point(self.global_pose['x'],\
			self.global_pose['y'], 0)
		msg.pose.pose.orientation = Quaternion(\
			*tf.transformations.quaternion_from_euler(0,0,self.global_pose['th']*(np.pi/180)))
		self.pub.publish(msg)

	def init_publisher(self):
		topic = 'Robot{:d}_odom'.format(self.id)
		self.pub = rospy.Publisher(topic, Odometry, queue_size=10)


class GlobalPose:
	def __init__(self):
		rospy.init_node('global_pose', anonymous=True)
		TCP_PORT = rospy.get_param("~wports", "10")
		TCP_PORT = map(int, TCP_PORT.split(","))
		self.cam_ids = [x%10 for x in TCP_PORT] 
		self.rate = rospy.get_param('~rate', 20)
		self.mother_cam = rospy.get_param('~mother_cam', 0)
		self.robot_ids = []
		self.robots = {}
		rospy.Subscriber('Robot_poses', CamPoseArray, self.get_robot_poses_cb)

	def generate_new_robots(self, msg, publish=True):
		new_robots_ids = [i.botID for i in msg.cp_list if i.botID not in self.robots]
		for id in new_robots_ids:
			self.robots[id] = Robot(id, self.tfs, publish)
			self.robot_ids = self.robot_ids + new_robots_ids

	def get_robot_poses_cb(self, msg, publish=True):
		self.generate_new_robots(msg, publish)
		for i in msg.cp_list:
			self.robots[i.botID].set_pose(i.pose, i.camID)
			self.robots[i.botID].gen_global_pose(self.mother_cam)			
			self.robots[i.botID].publish_odom()			
		self.robot_poses = msg

	def spin(self):
		rospy.loginfo("Finding Global Positions")
		rate = rospy.Rate(self.rate)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		rospy.loginfo("Shutting down Global Positions")
		rospy.sleep(1)

	def set_tf(self, filename, mother_cam):
		tfw = TFWrapper(filename, mother_cam, self.cam_ids)
		tfw.get_data_from_file()
		print tfw.graph
		tfw.create_tfs()
		self.tfs = tfw.tfs
		print 'tfs', self.tfs

def main():
	
	Global_Pose = GlobalPose();
	Global_Pose.set_tf('somefilename2', 0)
	Global_Pose.spin()

if __name__ == '__main__':
	main(); 
