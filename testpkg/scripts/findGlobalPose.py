#!/usr/bin/python
import rospy
import roslib
import math 
import numpy
import tf

from nav_msgs.msg import Odometry
from custom_msgs.msg import  CamPose, CamPoseArray
from geometry_msgs.msg import Point, Quaternion, Pose2D

class GlobalPose:
  def __init__(self):
	rospy.init_node('Global Positions')
	self.rate = rospy.get_param('~rate', 100)
	self.child_frame_id = rospy.get_param('~child_frame_id','/base_link')
	self.frame_id = rospy.get_param('~frame_id','/odom')
	rospy.Subscriber('Robot_poses', CamPoseArray, self.getRobotPosesCb)

	self.robot_poses = CamPoseArray()
	self.pose = {'x':0, 'y': 0, 'th': 0, 'cam': 0, 'bot':0}
	# self.cTrans01 = [2.65, 0]
	# self.cTrans02 = [2.65, 0]
	self.xaxis, self.yaxis, self.zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)
	self.camTrans0_1, self.camRot0_1 = (0, 2.65, 0), (0, 0, 0)
	self.camTrans0_2, self.camRot0_2 = (0, 2.65, 0), (0, 0, 0)
	self.eye = numpy.matrix([0, 0, 0, 1])  

  def update(self):
  	if len(self.robot_poses.cp_list) > 0:
	  	for loop in range(len(self.robot_poses.cp_list)):
	  		pose_temp = self.robot_poses.cp_list[loop]
		  	self.pose['x'] = pose_temp.pose.x
		  	self.pose['y'] = pose_temp.pose.y
		  	self.pose['th'] = pose_temp.pose.theta
		  	self.pose['cam'] = pose_temp.camID
		  	self.pose['bot'] = pose_temp.botID
		  	# print("hello")
		  	# print(self.pose)
		  	(self.pose['x'], self.pose['y'], self.pose['th']) = self.getGlobalPosition(self.pose)
		  	# print("wow")
		  	# print(self.pose)
		  	# print("end")
	  		odom_msg = self.pub_odometry(self.pose)
	  		self.odom_pub = rospy.Publisher('Robot'+str(self.pose['bot'])+'odom', Odometry, queue_size=10)
	  		self.odom_pub.publish(odom_msg)
	else:
		pass
		# print("Received Empty List")

  def pub_odometry(self,poses):
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = self.frame_id + str(poses['bot'])
    odom_msg.child_frame_id = self.child_frame_id
    odom_msg.pose.pose.position = Point(poses['x'], poses['y'], 0)
    odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,poses['th'] * (math.pi / 180)))
    return odom_msg

  def getRobotPosesCb(self, msg):
	self.robot_poses = msg
	

  def getGlobalPosition(self, robot_pose):
	if robot_pose['cam'] == 0:
		(x, y, theta) = (robot_pose['y'], robot_pose['x'], -robot_pose['th'])
	
	if robot_pose['cam'] == 1:
		if robot_pose['th'] >= 0:f.transformations.quaternion_from_euler(0,0,poses['th'] * (math.pi / 180)))
			th = 180 - robot_pose['th']
		if robot_pose['th'] < 0:
			th = -180 - robot_pose['th']
		(x, y, theta) = ( ( 0.170 - robot_pose['y'] ), (2.651- robot_pose['x']), th )

	if robot_pose['cam'] == 2:
		# print robot_pose['th']
		if robot_pose['th'] <= 90:
			th = -(90 + robot_pose['th'])
		else:
			th = 270 - robot_pose['th']
		(x, y, theta) = (robot_pose['x'] - 1.687, .272 - robot_pose['y'], th)

	return (x, y, theta)

 #  def getPose(self, robot_pose, camtrans):
	# T = tf.transformations.translation_matrix( camtrans )
	# C = tf.transformations.compose_matrix(angles= (0, 0, 0), translate= (robot_pose['y'], robot_pose['x'], 0))
	# D = tf.transformations.concatenate_matrices(T, C)
	# scale, shear, angles, translate, perspective = tf.transformations.decompose_matrix(D)
	# x = translate[0]
	# y = translate[1]
	# theta = angles[2]
	# return (x, y, theta)

  def spin(self):
	rospy.loginfo("Finding Global Positions")
	rate = rospy.Rate(self.rate)
	rospy.on_shutdown(self.shutdown)
	while not rospy.is_shutdown():
	  self.update();
	  rate.sleep()
	rospy.spin()

  def shutdown(self):
	rospy.loginfo("Shutting down Global Positions")
	rospy.sleep(1)

def main():
  Global_Pose = GlobalPose();
  Global_Pose.spin()

if __name__ == '__main__':
  main(); 
