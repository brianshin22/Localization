#!/usr/bin/env python

# can_sender.py
# reads in a data file and publishes, one at a time, the data needed for localization
# publishes to five topics: pose, speed, sas, left_lane, right_lane

import rospy
from std_msgs.msg import String, Float64
from localize.msg import LaneMeasure
from geometry_msgs.msg import Pose2D
#import UnpackCANData as ld
import LPF_UnpackCANData_WT as ld


def can_sender():
    
    GPS_accept = 5    
    
    pub1 = rospy.Publisher('pose', Pose2D, queue_size=10)
    pub2 = rospy.Publisher('speed_straight', Float64, queue_size=10)
    pub3 = rospy.Publisher('sas', Float64,  queue_size=10)
    pub4 = rospy.Publisher('speed_lateral', Float64, queue_size=10)
    pub5 = rospy.Publisher('dot_Psi', Float64,  queue_size=10)
    pub6 = rospy.Publisher('leftLane',LaneMeasure,queue_size=10)
    pub7 = rospy.Publisher('rightLane',LaneMeasure,queue_size=10)
    rospy.init_node('can_sender', anonymous=True)
    
    #filepath = '../../../../Data Files/CAN/CPG_Oval/t1.mat'
    #filepath = '/home/mpc/Localization/Data Files/CAN/CPG_Oval/t1.mat'
    filepath = '/home/mpc/Localization/Data Files/CAN/CPG_WindingTrack/WT_LSRL_clean.mat'
    
# calls custom function (from UnpackCANData) that reads CAN data)    
    #(time, X,Y,Psi,velocity,del_f,a_l0,a_l1,a_l2,a_l3,a_r0,a_r1,a_r2,a_r3,
    #a_l_quality,a_r_quality)=ld.loadCANdata(filepath)
    
    #(time, X,Y,Psi,velocity,del_f)=ld.load(filepath)
    frequency = 50
    GPS_Pos0 = [-118.0266,35.0536]    
    
    (time, X, Y, Psi, V_straight, del_f, V_lateral, dot_Psi, \
            a_r0, a_r1, a_r2, a_r3, a_r_quality, a_l0, a_l1, \
            a_l2, a_l3, a_l_quality)=ld.loadCANdata(filepath,frequency, GPS_Pos0)
    
      
    rate = rospy.Rate(frequency) # period = 1ms    
    #dt = time[1] - time[0]
    #filter_dt = 1./filter_rate
    #increment = filter_dt / dt
    
    i = 0
    rospy.loginfo('Begin loop')
    #rospy.loginfo(len(X))
    while (not rospy.is_shutdown() and i <=len(X)):
        pose = Pose2D(X[i],Y[i],Psi[i])
        v_straight = Float64(V_straight[i])
        delf = Float64(del_f[i])
        v_lat = Float64(V_lateral[i])
        dpsi = Float64(dot_Psi[i])
        
        left = LaneMeasure(a_l0[i],a_l1[i],a_l2[i],a_l3[i],a_l_quality[i])
        right = LaneMeasure(a_r0[i],a_r1[i],a_r2[i],a_r3[i],a_r_quality[i])
        
        rospy.loginfo(GPS_accept)
        
        rospy.loginfo(pose)
        rospy.loginfo(v_straight)
        rospy.loginfo(delf)
        rospy.loginfo(v_lat)
        rospy.loginfo(dpsi)
        rospy.loginfo(left)
        rospy.loginfo(right)
        
        #if i % GPS_accept == 0:    
        pub1.publish(pose)
        pub2.publish(v_straight)
        pub3.publish(delf)
        pub4.publish(v_lat)
        pub5.publish(dpsi)
        
        
        pub6.publish(left)
        pub7.publish(right)
        #i = i + increment
        i = i+1
        rate.sleep()

if __name__ == '__main__':    
    try:
        can_sender()
    except rospy.ROSInterruptException:
	pass
