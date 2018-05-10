#!/usr/bin/env python
import rospy
from custom_msgs.msg import CamPose

def talker():
    rospy.init_node('CT0')
    pub = rospy.Publisher('~custom_chatter', CamPose)
    r = rospy.Rate(10) #10hz
    msg = CamPose()
    msg.camID = 15
    msg.pose.x = 1
    msg.pose.y = 17
    msg.pose.theta = 10

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass