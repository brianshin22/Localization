#!/usr/bin/env python
import rospy
from location_service.srv import *
from std_msgs.msg import String, Float64
from localize.msg import Pose
import particle_filter as pf

#each value is updating in a real time
float X, Y, Psi, v, del_f, X_out, Y_out, Psi_out

def callback1(pose):
    X = pose.X
    Y = pose.Y
    Psi = pose.HeadingAngle
    rospy.loginfo(rospy.get_caller_id() + "X = %f, Y = %f, Psi = %f\n", X, Y, Psi)
def callback2(speed):
    v = speed.data
    rospy.loginfo(rospy.get_caller_id() + "Velocity = %f\n", v)
def callback3(sas):
    del_f = sas.data
    rospy.loginfo(rospy.get_caller_id() + "Steering Angle = %f\n", del_f)
def handle_location_request() 
#return the current X_out, Y_out, Psi_out everytime when recives a service request
    return PoseServiceResponse(X_out, Y_out, Psi_out)

def location_server():
    rospy.init_node('location_server')
    #initiate the sevice node
    s = rospy.Service('location_request', PoseService, handle_location_request)
    #initiate the subscriber nodes
    #each subscriber call the callback functions to update the variable value upon receiving the new value
    rospy.Subscriber("pose", Pose, callback1)
    rospy.Subscriber("speed", Float64, callback2)
    rospy.Subscriber("sas", Float64, callback3)
    # set the rate to run particle filter    
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
    #every time particle fillter runs will update the X_out,Y_out,Psi_out values
        (X_out,Y_out,Psi_out) = pf.particle_filter(num_particles,X,Y,Psi,v,del_f)
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    location_server()
