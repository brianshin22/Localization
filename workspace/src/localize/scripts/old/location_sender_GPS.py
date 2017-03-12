#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#

# location_sender_GPS.py
# calls GPS particle filter to provide filtered location estimate
# publishes to topic "location" a message of type Pose2D



import rospy
from std_msgs.msg import String, Float64, Int32
#from localize.msg import LaneMeasure
from geometry_msgs.msg import Pose2D
import particle_filter as pf
import logging

logging.basicConfig(filename='/home/mpc/Localization/workspace/src/localize/log/test1.log',level=logging.DEBUG)

X = 0.
Y = 0.
Psi = 0.
v = 0.
del_f = 0.
flag = 0 # flags for if there's GPS new measurement coming in for x, y and psi
#i = 0  # measurements counter (FOR TESTING PURPOSE ONLY)
#GPS_accept = 50 #measurement will be take every s measurements for robustness testing (FOR TESTING PURPOSE ONLY)


def callback1(pose):
    global X,Y,Psi,flag
    # global i
    #if i < GPS_accept:
        #i = i+1
   # else:
    X = pose.x
    Y = pose.y
    Psi = pose.theta
    # rospy.loginfo("Received GPS measurement.")
    
    # set flag to 1, indicating that GPS signal arrived the time step before
    # publishing rate period
    flag = 1
    #i = 0
    #rospy.loginfo(rospy.get_caller_id() + "X = %f, Y = %f, Psi = %f\n", X, Y, Psi)

def callback2(speed):
    global v
    v = speed.data
    #rospy.loginfo(rospy.get_caller_id() + "Velocity = %f\n", v)
def callback3(sas):
    global del_f
    del_f = sas.data
    #rospy.loginfo(rospy.get_caller_id() + "Steering Angle = %f\n", del_f)

def location_sender():
    global flag
    rospy.init_node('location_sender_GPS', anonymous=True)
    # once location_sender receives the data it uses callback functions to update the variables
    rospy.Subscriber("pose", Pose2D, callback1)    
    rospy.Subscriber("speed", Float64, callback2)
    rospy.Subscriber("sas", Float64, callback3)

    pub = rospy.Publisher('location', Pose2D, queue_size=10)
    
    # set rate here
    rate = 50
    rate_ROS = rospy.Rate(rate) # 10hz
    while not rospy.is_shutdown():
        (X_out,Y_out,Psi_out) = pf.particle_filter(flag,rate,X,Y,Psi,v,del_f) # particle filter runs

        pose_out = Pose2D(X_out,Y_out,Psi_out) #Assign particle filter output to the message for publishing
        pub.publish(pose_out) # Publish the output from particle filter
        # reset GPS arrived flag to 0
        flag=0
        rate_ROS.sleep()



if __name__ == '__main__':
    try:
        location_sender()
    except rospy.ROSInterruptException:
        pass
