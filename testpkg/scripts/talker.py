#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def talker():
    data = Twist()
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5) # 10hz
    while not rospy.is_shutdown():
        data.linear.x = random.random()
        data.angular.z = random.random()-.5
        hello_str = "Abhay--"
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass