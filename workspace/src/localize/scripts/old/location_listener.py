#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %f, %f, %f", data.x, data.y, data.theta)

def location_listener():
    rospy.init_node('location_listener', anonymous=True)
    rospy.Subscriber("location", Pose2D, callback)
     # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    location_listener()
