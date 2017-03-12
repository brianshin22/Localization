#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import numpy as np
from std_msgs.msg import String, Float64, Int32
from localize.msg import LaneMeasure
from geometry_msgs.msg import Pose2D
#import particle_filter as pf
import merged_filter as mf
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

# initialize dummy left and right lane measurements
left = (0,0,0,0,0)
right = (0,0,0,0,0)

def callback1(pose):
    global X,Y,Psi,flag
    # global i
    #if i < GPS_accept:
        #i = i+1
   # else:
    X = pose.x
    Y = pose.y
    Psi = pose.theta
    #rospy.loginfo("Received GPS measurement.")
    
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
def callback4(leftLane):
    global left
    left = (leftLane.a0,leftLane.a1,leftLane.a2,leftLane.a3,leftLane.quality)
    #rospy.loginfo(rospy.get_caller_id() + "Left lane measurements received\n")
def callback5(rightLane):
    global right
    right = (rightLane.a0,rightLane.a1,rightLane.a2,rightLane.a3,rightLane.quality)
    #rospy.loginfo(rospy.get_caller_id() + "Right lane measurements received\n")


def location_sender():
    global flag
    rospy.init_node('location_sender', anonymous=True)
    # once location_sender receives the data it uses callback functions to update the variables
    rospy.Subscriber("pose", Pose2D, callback1)    
    rospy.Subscriber("speed", Float64, callback2)
    rospy.Subscriber("sas", Float64, callback3)
    rospy.Subscriber("leftLane", LaneMeasure,callback4)
    rospy.Subscriber("rightLane",LaneMeasure,callback5)

    pub = rospy.Publisher('location', Pose2D, queue_size=10)
    pub2 = rospy.Publisher('runtime', Int32, queue_size=10)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        #(X_out,Y_out,Psi_out) = pf.particle_filter(num_particles,X,Y,Psi,v,del_f,flag)# praticle filter runs
        tic = rospy.get_rostime()
        (X_out,Y_out,Psi_out) = mf.merged_filter(flag,X,Y,Psi,v,del_f,left[0],left[1],left[2],left[3],
        right[0],right[1],right[2],right[3],left[4],right[4])
        toc = rospy.get_rostime()
        difference = (toc.nsecs - tic.nsecs)
        rospy.loginfo("%i\n",difference)
        dif = str(difference)
        #logging.info("%s\n",dif)
        pose_out = Pose2D(X_out,Y_out,Psi_out) #Assign particle filter output to the message for publishing
        #rospy.loginfo(pose_out) # log the output from particle filter
        t = rospy.get_rostime()         
		saveLine=str(t)+","+str(dif)+","+str(X_out)+","+str(Y_out)+","+str(Psi_out)+"\n"
        saveFile=open("PFdata.csv","a")
        saveFile.write(saveLine)
		saveFile.close()

	pub.publish(pose_out) # Publish the output from particle filter
        pub2.publish(difference)
        
        # reset GPS arrived flag to 0
        flag=0
        rate.sleep()

if __name__ == '__main__':
    try:
        location_sender()
    except rospy.ROSInterruptException:
        pass
