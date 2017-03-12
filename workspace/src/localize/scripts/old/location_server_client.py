#!/usr/bin/env python
import sys
import rospy
from location_service.srv import *

float X, Y, Psi

def location_server__client()
#calling the location_request service
    rospy.wait_for_service('location_request')

    try:
#location_request serves as a handle to the location_request service
        location_request = rospy.ServiceProxy('location_request', PoseService)
#pose gets update every time the service return a value
        pose=location_request()
        return pose
    except rospy.ServiceException, e:≈
        print "Service call failed: %s"%e

if __name__ == "__main__":
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(location_server_client())
        rate.sleep()
