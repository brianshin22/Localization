#!/usr/bin/env python

# insert srv import here

import rospy

def GPS_location_server():
    rospy.init_node('GPS_location_server')
    s = rospy.service('GPS_location', location, 
    rospy.spin()


if __name__ == "main":
    GPS_location_server()
