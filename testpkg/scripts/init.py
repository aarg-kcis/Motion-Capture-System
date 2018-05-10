#!/usr/bin/env python
import rospy
from calibrate import Calibrate
from cam_tracker import CamTracker
from custom_msgs.msg import CamPose, CamPoseArray


def talker(numcam, num_bots, tcp_ips, tcp_ports, buffer_size):
	if not (numcam == len(tcp_ips) == len(tcp_ports)):
		rospy.loginfo('The number of cameras and ports and ip don\'t match')
		return
	CAM_IDS = [x%10 for x in TCP_PORT] 

	trackers = {}
	for x, y in enumerate(CAM_IDS):
		print x, y
		z = CamTracker(y, num_bots, tcp_ips[x], tcp_ports[x], buffer_size)
		trackers[y] = z

	for i in trackers.keys():
		trackers[i].thread.start()
	bot_pub = rospy.Publisher('Robot_poses', CamPoseArray, queue_size=10)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		data = update(numcam, trackers)
		bot_pub.publish(data)
		rate.sleep()
	rospy.spin()

def update(numcam, trackers):
	data = CamPoseArray()
	bot_registered = []
	for i in trackers.keys():
		fetched_data = trackers[i].purana_pose
		for j in fetched_data.keys():
			if j not in bot_registered:
				x = CamPose()
				x.camID = i
				x.botID = j
				x.pose.x = fetched_data[j]['x']
				x.pose.y = fetched_data[j]['y']
				x.pose.theta = fetched_data[j]['theta']
				if not (x.pose.x < 0 or x.pose.y < 0):
					bot_registered.append(j)
					data.cp_list.append(x)
	return data

	

if __name__ == '__main__':
	rospy.init_node('custom_talker', anonymous=True)

	NUM_CAM = rospy.get_param("~numcam", 3)
	NUM_BOTS = rospy.get_param("~numbot", 6)
	TCP_PORT = rospy.get_param("~ports", "")
	TCP_IP = rospy.get_param("~ips", "")
	TCP_PORT = TCP_PORT.split(",")
	TCP_PORT = map(int, TCP_PORT)
	TCP_IP = [x.strip() for x in TCP_IP.split(',')]
	BUFFER_SIZE = 1024

	try:
		talker(NUM_CAM, NUM_BOTS, TCP_IP, TCP_PORT, BUFFER_SIZE)
	except rospy.ROSInterruptException: 
		pass
